classdef inflation_results
  
  properties
    e_history
    p_history
    start
    obstacles
    e_time
    p_time
    total_time
    iters
    n_obs
  end
  
  methods
    function obj = inflation_results()
      obj.e_history = {};
      obj.p_history = {};
      obj.start = [];
      obj.obstacles = {};
      obj.n_obs = 0;
      obj.e_time = 0;
      obj.p_time = 0;
      obj.total_time = 0;
      obj.iters = 0;
    end
  end
  
end

