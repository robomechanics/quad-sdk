function v = OP_MMIN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 98);
  end
  v = vInitialized;
end
