{ ... }:

{
  imports = [
    ./networking.nix
    ./administration.nix
  ];

  config = {
    boot.loader = {
      systemd-boot.enable = true;
      efi.canTouchEfiVariables = true;
    };
  };
}