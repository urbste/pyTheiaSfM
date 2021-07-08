#include <pybind11/pybind11.h>

#include "pytheia/io/io.h"
#include "pytheia/matching/matching.h"
#include "pytheia/math/math.h"
#include "pytheia/solvers/solvers.h"
#include "pytheia/sfm/sfm.h"

namespace pytheia {

PYBIND11_MODULE(pytheia, m) {

    m.doc() = "Python binding for TheiaSfM";
    // from Open3D repo:
    // The binding order matters: if a class haven't been binded, binding the
    // user of this class will result in "could not convert default argument
    // into a Python object" error.
    //utility::pybind_utility(m);

    // register all submodules here
    io::pytheia_io(m);
    matching::pytheia_matching(m);
    math::pytheia_math(m);
    sfm::pytheia_sfm(m);
    solvers::pytheia_solvers(m);
}

}  // namespace pytheia