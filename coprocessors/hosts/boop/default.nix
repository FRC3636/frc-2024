{ ... }:

{
  config = {
    networking = {
      hostName = "boop";

      robotNetwork = {
        interface = "wlp1s0";
        address = "10.36.36.11";
      };
    };

    system.stateVersion = "23.05";
  };
}
