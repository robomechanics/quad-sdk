function v = OP_NOT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 39);
  end
  v = vInitialized;
end
