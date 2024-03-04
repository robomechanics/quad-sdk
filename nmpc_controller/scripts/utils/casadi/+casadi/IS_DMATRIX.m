function v = IS_DMATRIX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 117);
  end
  v = vInitialized;
end
