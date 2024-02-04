{ ... }:

{
  imports = [
    ./hardware-configuration.nix
  ];

  config = {
    networking = {
      hostName = "tangerine";

      interfaces.enp1s0 = {
        ipv4 = {
          addresses = [{ address = "10.36.36.10"; prefixLength = 24; }];
          routes = [{ address = "0.0.0.0"; prefixLength = 0; via = "10.36.36.1"; }];
        };
      };
    };

    system.stateVersion = "23.05";
  };
}