function v = OP_SINH()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 53);
  end
  v = vInitialized;
end
