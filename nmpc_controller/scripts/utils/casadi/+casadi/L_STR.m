function v = L_STR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 13);
  end
  v = vInitialized;
end
