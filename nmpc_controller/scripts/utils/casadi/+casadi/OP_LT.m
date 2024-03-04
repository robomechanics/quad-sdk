function v = OP_LT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 35);
  end
  v = vInitialized;
end
