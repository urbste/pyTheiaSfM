from random_recon_gen import RandomReconGenerator
import pytheia as pt


def test_write_reconstruction_json():
    gen = RandomReconGenerator()
    gen.generate_random_recon()

    pt.io.WriteReconstructionJson(gen.recon, "test_recon.json")


if __name__ == "__main__":
    test_write_reconstruction_json()
