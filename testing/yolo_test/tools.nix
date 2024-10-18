{ lib
, buildPythonPackage
, fetchPypi
, setuptools
, wheel
}:

buildPythonPackage rec {

  pname = "ultralytics";
      version = "8.2.70";
      src = fetchPypi {
        inherit pname version;
	sha256 = "f23ec4a1d377109257a530c5022403766502df898133ede5b94d953adcf9e5bf"; 
      };

}
