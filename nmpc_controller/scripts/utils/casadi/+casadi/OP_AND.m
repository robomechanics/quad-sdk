function v = OP_AND()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 40);
  end
  v = vInitialized;
end
