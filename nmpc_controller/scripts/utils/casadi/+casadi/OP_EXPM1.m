function v = OP_EXPM1()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 110);
  end
  v = vInitialized;
end
