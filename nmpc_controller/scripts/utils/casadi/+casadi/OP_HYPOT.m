function v = OP_HYPOT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 111);
  end
  v = vInitialized;
end
