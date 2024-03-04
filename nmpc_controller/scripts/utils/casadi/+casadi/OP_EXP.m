function v = OP_EXP()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 22);
  end
  v = vInitialized;
end
