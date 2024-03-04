function v = OP_NORMINF()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 96);
  end
  v = vInitialized;
end
