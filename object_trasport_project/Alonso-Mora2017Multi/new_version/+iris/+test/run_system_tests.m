function run_system_tests()
iris.test.test_c_space_3d();
iris.test.test_grid_2d();
iris.test.test_grid_segmentation();
iris.test.test_points_2d;
iris.test.test_poly_2d();
iris.test.test_poly_3d();
iris.test.test_poly_Nd(4);
iris.test.test_polygon_recovery();
iris.test.test_simple_poly_2d();
iris.test.test_boundary();
if ~exist('gurobi')
  if exist('addpath_gurobi.m')
    addpath_gurobi()
  end
end
if exist('gurobi')
  disp('gurobi found, running additional tests')
  iris.test.test_uav_demo();
  iris.test.test_uav_demo('4d');
else
  disp('gurobi not found, skipping additional tests')
end
iris.test.test_thin_ellipsoid_simple();
disp('Tests complete');
end

