function v = SOLVER_RET_NAN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 4);
  end
  v = vInitialized;
end
