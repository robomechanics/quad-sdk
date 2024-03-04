function v = OP_CALL()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 64);
  end
  v = vInitialized;
end
