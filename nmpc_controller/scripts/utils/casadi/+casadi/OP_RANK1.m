function v = OP_RANK1()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 75);
  end
  v = vInitialized;
end
