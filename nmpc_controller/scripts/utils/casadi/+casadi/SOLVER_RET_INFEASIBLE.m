function v = SOLVER_RET_INFEASIBLE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 5);
  end
  v = vInitialized;
end
