function v = OP_VERTCAT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 77);
  end
  v = vInitialized;
end
