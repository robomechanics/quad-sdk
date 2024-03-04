function v = OP_CONVEXIFY()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 107);
  end
  v = vInitialized;
end
