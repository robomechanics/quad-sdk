function v = OP_TRANSPOSE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 70);
  end
  v = vInitialized;
end
