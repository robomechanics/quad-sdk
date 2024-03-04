function v = LABEL()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 14);
  end
  v = vInitialized;
end
