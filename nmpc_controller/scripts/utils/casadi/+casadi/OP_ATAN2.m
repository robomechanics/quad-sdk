function v = OP_ATAN2()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 59);
  end
  v = vInitialized;
end
