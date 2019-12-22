classdef Polyhedron

  properties
    A;
    b;
    Aeq = [];
    beq = [];
    vertices;
    has_vertices = false;
  end

  methods
    function obj = Polyhedron(A, b, Aeq, beq)
      if nargin < 3
        Aeq = [];
      end
      if nargin < 4
        beq = [];
      end
      obj.A = A;
      obj.b = b;
      obj.Aeq = Aeq;
      obj.beq = beq;
    end

    function vertices = getVertices(obj)
      obj = obj.reduce();
      if ~obj.has_vertices
        if exist('cddmex', 'file')
          H = struct('A', [obj.Aeq; obj.A], 'B', [obj.beq; obj.b], 'lin', (1:size(obj.Aeq, 1))');
          V = cddmex('extreme', H);
          obj.vertices = V.V';
        else
          obj.vertices = iris.thirdParty.polytopes.lcon2vert(obj.A, obj.b, obj.Aeq, obj.beq)';
        end
        obj.has_vertices = true;
      end
      vertices = obj.vertices;
    end
    
    function reduced_poly = reduce(obj)
      % Find a minimal representation of the polyhedron
      if ~exist('cddmex', 'file')
        error('IRIS:MissingDependency', 'This function requires the cddmex tool. The easiest way to get it is using tbxmanager: http://www.tbxmanager.com/');
      end
      H = struct('A', [obj.Aeq; obj.A], 'B', [obj.beq; obj.b], 'lin', (1:size(obj.Aeq, 1))');
      Hred = cddmex('reduce_h', H);
      assert(isempty(Hred.lin), 'as far as I know, Hred.lin should always be empty. That is, the reduced polyhedron should not contain equality constraints. -rdeits');
      reduced_poly = iris.Polyhedron(Hred.A, Hred.B);
    end

    function plotVertices(obj, varargin)
      vertices = obj.getVertices();
      if ~isempty(vertices)
        if size(vertices, 2) > 2
          k = convhull(vertices(1,:), vertices(2,:));
        else
          k = [1:size(vertices, 2), 1];
        end
        plot(vertices(1,k), vertices(2,k), varargin{:});
      end
    end

    function drawLCMGL(obj, lcmgl)
      lcmgl.glBegin(lcmgl.LCMGL_LINES);
      vertices = obj.getVertices();
      if size(vertices, 2) > 2
        k = convhull(vertices(1,:), vertices(2,:));
      else
        k = [1:size(vertices, 2), 1];
      end
      for j = 1:length(k)-1
        lcmgl.glVertex3d(vertices(1,k(j)), vertices(2,k(j)), vertices(3,k(j)));
        lcmgl.glVertex3d(vertices(1,k(j+1)), vertices(2,k(j+1)), vertices(3,k(j+1)));
      end
      lcmgl.glEnd();
    end
    
    function obj = normalize(obj)
      n = zeros(size(obj.A, 1), 1);
      for j = 1:size(obj.A, 1)
        n(j) = norm(obj.A(j,:));
        obj.A(j,:) = obj.A(j,:) / n(j);
        obj.b(j) = obj.b(j) / n(j);
      end
      obj.A = obj.A(n > 0, :);
      obj.b = obj.b(n > 0);
      
      n = zeros(size(obj.Aeq, 1), 1);
      for j = 1:size(obj.Aeq, 1)
        n(j) = norm(obj.Aeq(j,:));
        obj.Aeq(j,:) = obj.Aeq(j,:) / n(j);
        obj.beq(j) = obj.beq(j) / n(j);
      end
      obj.Aeq = obj.Aeq(n > 0, :);
      obj.beq = obj.beq(n > 0);
    end

  end

  methods(Static)
    function obj = fromVertices(vertices)
      [A, b] = iris.thirdParty.polytopes.vert2lcon(vertices');
      obj = iris.Polyhedron(A, b);
    end

    function obj = from2DVertices(vertices)
      assert(size(vertices, 1) == 2);
      x = vertices(1,:);
      y = vertices(2,:);
      k = convhull(x,y, 'simplify', true);
      A = [(y(k(2:end)) - y(k(1:end-1)))', (x(k(1:end-1)) - x(k(2:end)))'];
      b = sum(A' .* [x(k(1:end-1)); y(k(1:end-1))], 1)';
      obj = iris.Polyhedron(A, b);
    end

    function obj = from2DVerticesAndPlane(vertices, normal, v)
      assert(size(vertices, 1) == 2);
      % normal' * [x;y;z] = v;
      obj = iris.Polyhedron.from2DVertices(vertices);
      obj.Aeq = reshape(normal, 1, []);
      obj.beq = v;
      obj.A = [obj.A, zeros(size(obj.A, 1), 1)];
    end

    function obj = fromBounds(lb, ub)
      % create a polyhedron representing a bounding box in n dimensions
      dim = length(lb);
      assert(length(lb) == length(ub));
      A = [eye(dim); -eye(dim)];
      b = [reshape(ub,[],1); reshape(-lb,[],1)];
      obj = iris.Polyhedron(A, b);
    end
  end
end
