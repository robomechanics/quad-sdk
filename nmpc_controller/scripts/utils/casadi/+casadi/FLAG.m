function v = FLAG()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 122);
  end
  v = vInitialized;
end
