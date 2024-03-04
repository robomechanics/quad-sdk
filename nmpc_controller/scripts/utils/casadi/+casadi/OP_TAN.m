function v = OP_TAN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 31);
  end
  v = vInitialized;
end
