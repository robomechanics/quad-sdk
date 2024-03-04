function v = IS_MEMBER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 115);
  end
  v = vInitialized;
end
