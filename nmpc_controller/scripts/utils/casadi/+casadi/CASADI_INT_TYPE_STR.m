function v = CASADI_INT_TYPE_STR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 0);
  end
  v = vInitialized;
end
