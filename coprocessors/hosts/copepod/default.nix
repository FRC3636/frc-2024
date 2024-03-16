{ pkgs, ... }:

{
  imports = [ ./hardware-configuration.nix ];

  boot = {
    loader = {
      # Use the extlinux bootloader instead of Grub.
      grub.enable = false;
      generic-extlinux-compatible.enable = true;
    };
    kernelModules = [ "spidev" ];
  };
  systemd.user.services.rgb-2024 = {
    description = "RGB LED strip driver";
    wantedBy = [ "multi-user.target" ];
    after = [ "network.target" ];

    serviceConfig = {
      Type = "simple";
      ExecStart = "${pkgs.rgb-2024}/bin/rgb-2024";
      Restart = "on-failure";
    };
  };

  hardware = {
    raspberry-pi."4".apply-overlays-dtmerge.enable = true;
    deviceTree = {
      enable = true;
      filter = "*rpi-4-*.dtb";
      overlays = [{
        name = "spi";
        dtsText = ''
                  /dts-v1/;
          /plugin/;

          / {
          	compatible = "raspberrypi";
          	fragment@0 {
          		target = <&spi>;
          		__overlay__ {
          			cs-gpios = <&gpio 8 1>, <&gpio 7 1>;
          			status = "okay";
          			pinctrl-names = "default";
          			pinctrl-0 = <&spi0_pins &spi0_cs_pins>;
          			#address-cells = <1>;
          			#size-cells = <0>;
          			spidev@0 {
          				reg = <0>;	// CE0
          				spi-max-frequency = <500000>;
          				compatible = "spidev";
          			};

          			spidev@1 {
          				reg = <1>;	// CE1
          				spi-max-frequency = <500000>;
          				compatible = "spidev";
          			};
          		};
          	};
                  fragment@1 {
          		target = <&alt0>;
          		__overlay__ {
          			// Drop GPIO 7, SPI 8-11
          			brcm,pins = <4 5>;
          		};
          	};

          	fragment@2 {
          		target = <&gpio>;
          		__overlay__ {
          			spi0_pins: spi0_pins {
          				brcm,pins = <9 10 11>;
          				brcm,function = <4>; // alt0
          			};
          			spi0_cs_pins: spi0_cs_pins {
          				brcm,pins = <8 7>;
          				brcm,function = <1>; // out
          			};
          		};
          	};
          };
        '';
      }];
    };
  };

  users.groups.spi = { };
  users.groups.gpio = { };

  services.udev.extraRules = ''
    SUBSYSTEM=="spidev", KERNEL=="spidev0.0", GROUP="spi", MODE="0660"
    SUBSYSTEM=="bcm2835-gpiomem", KERNEL=="gpiomem", GROUP="gpio",MODE="0660"
    SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", RUN+="${pkgs.bash}/bin/bash -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
    SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add",RUN+="${pkgs.bash}/bin/bash -c 'chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"
  '';

  networking = {
    hostName = "copepod";

    robotNetwork = {
      interface = "end0";
      address = "10.36.36.20";
    };
  };

  users.users.root = { extraGroups = [ "spi" "gpio" ]; };

  environment.systemPackages = with pkgs; [
    libraspberrypi
    raspberrypi-eeprom
    rgb-2024
  ];

  nix.settings.trusted-users = [ "root" ];

  system.stateVersion = "23.05";
}

