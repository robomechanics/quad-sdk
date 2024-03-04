function v = OP_LE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 36);
  end
  v = vInitialized;
end
