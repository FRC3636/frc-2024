{ ... }:

{
  imports = [
    ./hardware-configuration.nix
  ];

  config = {
    networking = {
      hostName = "tangerine";

      robotNetwork = {
        interface = "enp1s0";
        address = "10.36.36.10";
      };
    };

    system.stateVersion = "23.05";
  };
}