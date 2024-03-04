function v = OP_NORM2()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 94);
  end
  v = vInitialized;
end
