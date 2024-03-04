function v = OPTI_UNKNOWN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 151);
  end
  v = vInitialized;
end
