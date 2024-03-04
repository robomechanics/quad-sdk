function v = SWIG_IND1()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 15);
  end
  v = vInitialized;
end
