function v = OPTI_PAR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 153);
  end
  v = vInitialized;
end
