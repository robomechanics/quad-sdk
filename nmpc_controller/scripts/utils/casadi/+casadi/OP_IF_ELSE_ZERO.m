function v = OP_IF_ELSE_ZERO()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 48);
  end
  v = vInitialized;
end
