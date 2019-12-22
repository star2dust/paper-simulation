function isequal = equal_up_to_permutations(A, B, tolerance)
% Returns true iff A and B are equal (to within tolerance) up to permutations of their rows. Greedily searches for matches row-by-row, which means that it may be possible for this function to return a false negative if several rows differ by amounts on the order of [tolerance]. 

if length(size(A)) ~= length(size(B)) || any(size(A) ~= size(B))
  error('Sizes of A and B must match');
end

B_rows_used = false(size(B, 1), 1);

for j = 1:size(A, 1)
  available_rows = find(~B_rows_used);
  [max_diff, best_row] = min(max(abs(bsxfun(@minus, A(j,:), B(available_rows,:))), [], 2));
  if max_diff > tolerance
    isequal = false;
    return;
  end
  B_rows_used(available_rows(best_row)) = true;
end

isequal = true;
return;
