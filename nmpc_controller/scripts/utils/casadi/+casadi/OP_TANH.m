function v = OP_TANH()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 55);
  end
  v = vInitialized;
end
