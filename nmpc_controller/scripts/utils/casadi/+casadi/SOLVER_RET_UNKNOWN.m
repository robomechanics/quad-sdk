function v = SOLVER_RET_UNKNOWN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 2);
  end
  v = vInitialized;
end
