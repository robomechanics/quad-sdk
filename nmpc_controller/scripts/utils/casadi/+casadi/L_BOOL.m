function v = L_BOOL()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 8);
  end
  v = vInitialized;
end
