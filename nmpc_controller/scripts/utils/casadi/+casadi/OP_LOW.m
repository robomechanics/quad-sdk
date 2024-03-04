function v = OP_LOW()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 66);
  end
  v = vInitialized;
end
