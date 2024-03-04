function v = SOLVER_RET_LIMITED()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 3);
  end
  v = vInitialized;
end
