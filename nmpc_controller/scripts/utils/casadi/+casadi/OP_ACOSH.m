function v = OP_ACOSH()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 57);
  end
  v = vInitialized;
end
