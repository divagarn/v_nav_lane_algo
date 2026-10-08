{
   "format_version_major" : 1,
   "format_version_minor" : 0,
   "format_version_patch" : 0,
   "name"                 : "auto_interface_rrm_command",
   "section"              : 7000,
   "comment"              : "Maintain compatible section 1000 commands",
   "commands"             : [
      {
         "id"            : 1,
         "name"          : "sensor_reboot",
         "comment"       : "Performs sensor reboot.",
         "public"        : true,
         "level"         : 0,
         "argument"      : "cmd_value"
      },
      {
         "id"            : 10,
         "name"          : "soft_sensor_reset",
         "comment"       : "Performs soft reset.",
         "public"        : true,
         "level"         : 0,
         "argument"      : "cmd_value"
      },
      {
         "id"            : 11,
         "name"          : "restore_default_param",
         "comment"       : "Restore default values in RAM and EEPROM.",
         "public"        : true,
         "level"         : 0,
         "argument"      : "section"
      },
      {
         "id"            : 20,
         "name"          : "sw_crc_check",
         "comment"       : "Perform crc check",
         "public"        : true,
         "level"         : 0,
         "argument"      : "fraction_of_second"
      }
   ]
}

