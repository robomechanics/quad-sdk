function v = SOLVER_RET_SUCCESS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 1);
  end
  v = vInitialized;
end
