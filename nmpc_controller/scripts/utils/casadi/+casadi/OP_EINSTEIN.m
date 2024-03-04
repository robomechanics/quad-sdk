function v = OP_EINSTEIN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 105);
  end
  v = vInitialized;
end
