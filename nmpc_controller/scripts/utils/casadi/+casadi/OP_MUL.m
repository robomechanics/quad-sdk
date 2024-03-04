function v = OP_MUL()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 19);
  end
  v = vInitialized;
end
