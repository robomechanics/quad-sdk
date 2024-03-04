function v = OP_OR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 41);
  end
  v = vInitialized;
end
