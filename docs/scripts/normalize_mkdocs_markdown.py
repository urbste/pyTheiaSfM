#!/usr/bin/env python3
"""Post-process pandoc RST->MD output for MkDocs Material."""
from __future__ import annotations

import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Union

BlockPart = Union[str, "Block"]


@dataclass
class Block:
    colons: int
    kind: str
    parts: List[BlockPart] = field(default_factory=list)


OPEN_RE = re.compile(r"^(:{3,})\s+(.+)$")


def parse_block(lines: List[str], i: int) -> tuple[Block, int]:
    m = OPEN_RE.match(lines[i])
    if not m:
        raise ValueError(f"expected open at {i}: {lines[i]!r}")
    n = len(m.group(1))
    kind = m.group(2).strip()
    i += 1
    parts: List[BlockPart] = []
    current: List[str] = []
    while i < len(lines):
        line = lines[i]
        if re.fullmatch(rf":{{{n}}}", line):
            if current:
                parts.append("\n".join(current).strip())
                current = []
            return Block(n, kind, parts), i + 1
        om = OPEN_RE.match(line)
        if om:
            n2 = len(om.group(1))
            # Pandoc may nest with fewer *or* more colons than the parent.
            if n2 != n:
                if current:
                    parts.append("\n".join(current).strip())
                    current = []
                child, i = parse_block(lines, i)
                parts.append(child)
                continue
        current.append(line)
        i += 1
    raise ValueError(f"unclosed block {kind} starting near line")


def render_note_inner(parts: List[BlockPart]) -> str:
    title = "Note"
    body_chunks: List[str] = []
    for p in parts:
        if isinstance(p, Block) and p.kind == "title" and p.parts:
            t0 = p.parts[0]
            if isinstance(t0, str):
                title = t0.strip()
        elif isinstance(p, str):
            body_chunks.append(p)
        elif isinstance(p, Block):
            body_chunks.append(render_block(p))
    body = "\n\n".join(x for x in body_chunks if x).strip()
    lines = [f'!!! note "{title}"', ""]
    for bl in body.split("\n"):
        lines.append(f"    {bl}" if bl else "")
    return "\n".join(lines).rstrip() + "\n\n"


def render_block(b: Block) -> str:
    if b.kind == "default-domain":
        return ""
    if b.kind.startswith("{") or b.kind == "{#citations}":
        return "\n\n".join(render_block_part(p) for p in b.parts).strip() + "\n\n"

    texts = [p for p in b.parts if isinstance(p, str)]
    nested = [p for p in b.parts if isinstance(p, Block)]

    if b.kind == "function":
        sig = texts[0].split("\n", 1)[0].strip() if texts else ""
        rest = "\n\n".join(
            t.split("\n", 1)[1].strip() if "\n" in t else ("" if t == sig else t)
            for t in texts
        ).strip()
        # merge: first part may be "sig\n\ndesc"
        if texts:
            full = texts[0]
            if "\n" in full:
                sig, _, tail = full.partition("\n")
                sig = sig.strip()
                rest = (tail.strip() + "\n\n" + rest).strip() if rest else tail.strip()
        out = f"**`{sig}`**\n\n"
        note_md = ""
        for nb in nested:
            if nb.kind == "note":
                note_md = render_note_inner(nb.parts)
            else:
                note_md += render_block(nb)
        out += note_md + rest + "\n\n"
        return out

    if b.kind == "note":
        return render_note_inner(b.parts)

    if b.kind == "title":
        return ""

    if b.kind in ("member",):
        lines = "\n\n".join(texts).strip()
        return f"- {lines}\n\n"

    if b.kind == "tabs":
        return render_material_tabs(b)

    if b.kind == "tab":
        # handled inside tabs
        return "\n\n".join(render_block_part(p) for p in b.parts)

    # class-like wrappers: View, Track, Camera, etc.
    inner = "\n\n".join(render_block_part(p) for p in b.parts).strip()
    return inner + "\n\n" if inner else ""


def render_material_tabs(b: Block) -> str:
    chunks: List[str] = []
    for p in b.parts:
        if not isinstance(p, Block) or p.kind != "tab":
            continue
        label = "Tab"
        body_parts: List[str] = []
        for q in p.parts:
            if isinstance(q, str):
                body_parts.append(q)
        body = "\n".join(body_parts).strip()
        first = body.split("\n", 1)[0].strip()
        rest = body.split("\n", 1)[1] if "\n" in body else ""
        if first in ("Python", "C++", "C"):
            label = first
            content = rest.strip()
        else:
            content = body
        chunks.append(f'=== "{label}"\n\n')
        for ln in content.split("\n"):
            chunks.append(f"    {ln}\n")
        chunks.append("\n")
    return "".join(chunks).rstrip() + "\n\n"


def render_block_part(p: BlockPart) -> str:
    if isinstance(p, str):
        return p
    return render_block(p)


def transform_divs(text: str) -> str:
    lines = text.splitlines()
    out: List[str] = []
    i = 0
    while i < len(lines):
        if OPEN_RE.match(lines[i]):
            blk, j = parse_block(lines, i)
            rendered = render_block(blk).strip()
            if rendered:
                out.append(rendered)
            i = j
        else:
            out.append(lines[i])
            i += 1
    return "\n".join(out)


def fix_interpreted_roles(text: str) -> str:
    def repl_class(m: re.Match) -> str:
        return f"`{m.group(1)}`"

    text = re.sub(
        r"`([^`]+)`\{\.interpreted-text role=\"class\"\}",
        repl_class,
        text,
    )
    text = re.sub(
        r"`([^`]+)`\{\.interpreted-text role=\"func\"\}",
        repl_class,
        text,
    )
    text = re.sub(
        r"`([^`]+)`\{\.interpreted-text role=\"eq\"\}",
        r"`\1`",
        text,
    )

    doc_pages = {
        "building": "building.md",
        "python_wrapper": "python_wrapper.md",
        "api": "api.md",
    }

    def repl_doc(m: re.Match) -> str:
        name = m.group(1)
        target = doc_pages.get(name, f"{name}.md")
        return f"[{name.replace('_', ' ')}]({target})"

    text = re.sub(
        r"`([^`]+)`\{\.interpreted-text role=\"doc\"\}",
        repl_doc,
        text,
    )

    refs = {
        "chapter-building": ("building.md", "chapter-building"),
        "chapter-contributing": ("contributions.md", "chapter-contributing"),
        "chapter-sfm": ("sfm.md", "chapter-sfm"),
        "chapter-cameras": ("cameras.md", "chapter-cameras"),
    }

    def repl_ref(m: re.Match) -> str:
        key = m.group(1)
        if key in refs:
            f, a = refs[key]
            return f"[{key}]({f}#{a})"
        return f"`{key}`"

    text = re.sub(
        r"`([^`]+)`\{\.interpreted-text role=\"ref\"\}",
        repl_ref,
        text,
    )
    return text


def fix_citations(text: str) -> str:
    def repl(m: re.Match) -> str:
        key = m.group(1)
        return f"[{key}](bibliography.md#{key})"

    return re.sub(r"\[\\\[(\w+)\\\]\]\(\)", repl, text)


def fix_title_ref(text: str) -> str:
    return re.sub(r"\[([^\]]+)\]\{\.title-ref\}", r"`\1`", text)


def strip_toctree(text: str) -> str:
    text = re.sub(
        r"^::: \{\.toctree[^}]*\}\n[^\n]*\n:::\s*",
        "",
        text,
        flags=re.MULTILINE,
    )
    return text


def fix_bibliography(text: str) -> str:
    """Simplify pandoc citation divs to headings with ids."""
    text = re.sub(r"^::: \{#citations\}\s*$", "", text, flags=re.MULTILINE)
    text = re.sub(
        r"^\[([^\]]+)\]\{#(\w+) \.citation-label\}\s*$",
        r"## \1 {#\2}",
        text,
        flags=re.MULTILINE,
    )
    return text


STRIP_BLOCKQUOTE_PREFIX = frozenset({"pose.md", "sfm.md", "ransac.md"})


def strip_gt_prefix(text: str, filename: str) -> str:
    """Pandoc sometimes wraps API blocks in reST blockquotes; strip leading '> '."""
    if filename not in STRIP_BLOCKQUOTE_PREFIX:
        return text
    out: List[str] = []
    for line in text.splitlines():
        if line.startswith("> "):
            out.append(line[2:])
        elif line == ">":
            out.append("")
        else:
            out.append(line)
    return "\n".join(out)


def process_file(path: Path) -> None:
    raw = path.read_text(encoding="utf-8")
    text = strip_gt_prefix(raw, path.name)
    text = strip_toctree(text)
    text = transform_divs(text)
    text = fix_interpreted_roles(text)
    text = fix_citations(text)
    text = fix_title_ref(text)
    if path.name == "bibliography.md":
        text = fix_bibliography(text)
    # unwrap accidental doubled newlines
    text = re.sub(r"\n{4,}", "\n\n\n", text)
    path.write_text(text.strip() + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[1] / "content"
    if not root.is_dir():
        print("content dir missing", root, file=sys.stderr)
        sys.exit(1)
    if len(sys.argv) > 1:
        for name in sys.argv[1:]:
            process_file(root / name)
        print("processed", len(sys.argv) - 1, "file(s)")
    else:
        for md in sorted(root.glob("*.md")):
            process_file(md)
        print("processed", len(list(root.glob("*.md"))), "files")


if __name__ == "__main__":
    main()
