function v = OP_FIND()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 65);
  end
  v = vInitialized;
end
