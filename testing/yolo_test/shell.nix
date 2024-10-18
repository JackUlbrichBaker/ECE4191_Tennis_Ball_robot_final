# shell.nix
let
  pkgs = import <nixpkgs> {};

    python = pkgs.python3.override {
    self = python;
    packageOverrides = pyself: pyfinal: {
      toolz = pyfinal.callPackage ./tools.nix { };
    };
  };


in pkgs.mkShell {
  packages = [
    (pkgs.python3.withPackages (python-pkgs: [
      python-pkgs.requests
      python-pkgs.opencv4
      python-pkgs.torchvision
      python-pkgs.gitpython
      python-pkgs.matplotlib
      python-pkgs.numpy
      python-pkgs.pillow
      python-pkgs.psutil
      #python-pkgs.PyYAML
      python-pkgs.requests
      python-pkgs.scipy
      python-pkgs.thop
      python-pkgs.torch
      python-pkgs.torchvision
      python-pkgs.tqdm
      python-pkgs.ultralytics
    ]))
  ];
}
