#include <mex.h>
#include <Eigen/Core>
#include <chrono>
#include "iris/iris.h"

using namespace Eigen;

// mxGetPrSafe and eigenToMatlab lovingly ripped off from the Drake toolbox: https://github.com/RobotLocomotion/drake
double * mxGetPrSafe(const mxArray *pobj) {
  if (!mxIsDouble(pobj)) mexErrMsgIdAndTxt("Iris:mxGetPrSafe:WrongType", "mxGetPr can only be called on arguments which correspond to Matlab doubles");
  return mxGetPr(pobj);
}

mxArray* eigenToMatlab(const MatrixXd &m)
{
  // this avoids zero initialization that would occur using mxCreateDoubleMatrix with nonzero dimensions.
  // see https://classes.soe.ucsc.edu/ee264/Fall11/cmex.pdf, page 8
  mxArray* pm = mxCreateDoubleMatrix(0, 0, mxREAL);
  int rows = static_cast<int>(m.rows());
  int cols = static_cast<int>(m.cols());
  int numel = rows * cols;
  mxSetM(pm, rows);
  mxSetN(pm, cols);
  if (numel)
    mxSetData(pm, mxMalloc(sizeof(double) * numel));
  memcpy(mxGetPr(pm), m.data(), sizeof(double)* numel);
  return pm;
}

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // auto begin = std::chrono::high_resolution_clock::now();

  if (nrhs != 5 || nlhs > 6) {
    mexErrMsgTxt("usage: [A, b, C, d, p_history, e_history] = inflate_regionmex(obstacles, A_bounds, b_bounds, start, options)");
  }

  int narg = 0;

  const mxArray* obstacles = prhs[narg];
  if (!mxIsCell(obstacles)) {
    mexErrMsgTxt("obstacles should be a cell array of matrices");
  }
  narg++;

  Map<MatrixXd> A_bounds(mxGetPrSafe(prhs[narg]), 
                         mxGetM(prhs[narg]),
                         mxGetN(prhs[narg]));
  narg++;
  Map<VectorXd> b_bounds(mxGetPrSafe(prhs[narg]),
                         mxGetNumberOfElements(prhs[narg]));
  narg++;

  Map<VectorXd> start(mxGetPrSafe(prhs[narg]),
                      mxGetNumberOfElements(prhs[narg]));
  narg++;

  // mexPrintf("made maps\n");

  const mxArray* options_ptr = prhs[narg];

  const int dim = start.size();
  if (A_bounds.cols() != dim) {
    mexPrintf("Problem dimension was inferred to be %d from length of [start], so [A_bounds] should have %d columns\n", start.size(), A_bounds.cols());
    mexErrMsgTxt("Dimension of problem does not match size of A_bounds");
  }
  if (A_bounds.rows() != b_bounds.size()) {
    mexErrMsgTxt("A_bounds and b_bounds should have the same number of rows");
  }

  iris::IRISProblem problem(dim);
  problem.setSeedPoint(start);

  // mexPrintf("set seed\n");

  const size_t n_obs = mxGetNumberOfElements(obstacles);
  for (size_t i=0; i < n_obs; i++) {
    const mxArray *obs_ptr = mxGetCell(obstacles, i);
    Map<MatrixXd> obs(mxGetPrSafe(obs_ptr),
                      mxGetM(obs_ptr),
                      mxGetN(obs_ptr));
    if (obs.rows() != dim) {
      mexErrMsgTxt("Each obstacle should be of size M x N where M is the dimension of [start].");
    }
    problem.addObstacle(obs);
  }

  // mexPrintf("added obstacles\n");

  iris::IRISOptions options;
  const mxArray *opt;
  opt = mxGetField(options_ptr, 0, "require_containment");
  if (opt) options.require_containment = static_cast<bool>(mxGetScalar(opt));

  opt = mxGetField(options_ptr, 0, "error_on_infeasible_start");
  if (opt) options.error_on_infeasible_start = static_cast<bool>(mxGetScalar(opt));

  opt = mxGetField(options_ptr, 0, "termination_threshold");
  if (opt) options.termination_threshold = mxGetScalar(opt);

  opt = mxGetField(options_ptr, 0, "iter_limit");
  if (opt) options.iter_limit = static_cast<int>(mxGetScalar(opt));

  // mexPrintf("got options\n");

  problem.setBounds(iris::Polyhedron(A_bounds, b_bounds));

  // auto end = std::chrono::high_resolution_clock::now();
  // auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(end - begin);
  // std::cout << "pre-solve time: " << elapsed.count() << " s" << std::endl;

  iris::IRISRegion region;
  std::unique_ptr<iris::IRISDebugData> debug;
  // begin = std::chrono::high_resolution_clock::now();
  try {
    if (nlhs > 4) {
      debug.reset(new iris::IRISDebugData());
      region = iris::inflate_region(problem, options, debug.get());
    } else {
      region = iris::inflate_region(problem, options);
    }
  } catch (iris::InitialPointInfeasibleError &exception) {
    mexErrMsgIdAndTxt("IRIS:InfeasibleStart", "Initial point is infeasible");
  }
  // end = std::chrono::high_resolution_clock::now();
  // elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(end - begin);
  // std::cout << "solve time: " << elapsed.count() << " s" << std::endl;

  // begin = std::chrono::high_resolution_clock::now();
  // mexPrintf("ran iris\n");

  narg = 0;
  if (nlhs > narg) plhs[narg] = eigenToMatlab(region.polyhedron.getA());
  narg++;

  if (nlhs > narg) plhs[narg] = eigenToMatlab(region.polyhedron.getB());
  narg++;

  if (nlhs > narg) plhs[narg] = eigenToMatlab(region.ellipsoid.getC());
  narg++;

  if (nlhs > narg) plhs[narg] = eigenToMatlab(region.ellipsoid.getD());
  narg++;

  if (nlhs > narg) {
    const size_t n_polys[1] = {debug->polyhedron_history.size()};
    plhs[narg] = mxCreateCellArray(1, n_polys);
    for (int i=0; i < debug->polyhedron_history.size(); i++) {
      const size_t dims[1] = {1};
      const char* fields[2] = {"A", "b"};
      mxArray* entry = mxCreateStructArray(1, dims, 2, fields);
      mxSetField(entry, 0, "A", eigenToMatlab(debug->polyhedron_history[i].getA()));
      mxSetField(entry, 0, "b", eigenToMatlab(debug->polyhedron_history[i].getB()));
      mxSetCell(plhs[narg], i, entry);
    }
  }
  narg++;

  if (nlhs > narg) {
    const size_t n_ellipsoids[1] = {debug->ellipsoid_history.size()};
    plhs[narg] = mxCreateCellArray(1, n_ellipsoids);
    for (int i=0; i < debug->ellipsoid_history.size(); i++) {
      const size_t dims[1] = {1};
      const char* fields[2] = {"C", "d"};
      mxArray* entry = mxCreateStructArray(1, dims, 2, fields);
      mxSetField(entry, 0, "C", eigenToMatlab(debug->ellipsoid_history[i].getC()));
      mxSetField(entry, 0, "d", eigenToMatlab(debug->ellipsoid_history[i].getD()));
      mxSetCell(plhs[narg], i, entry);
    }
  }
  narg++;

  // end = std::chrono::high_resolution_clock::now();
  // elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(end - begin);
  // std::cout << "post-solve time: " << elapsed.count() << " s" << std::endl;
}
