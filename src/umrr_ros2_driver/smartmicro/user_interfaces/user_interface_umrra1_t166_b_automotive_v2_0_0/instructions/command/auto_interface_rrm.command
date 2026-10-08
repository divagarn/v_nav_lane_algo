{
   "format_version_major" : 1,
   "format_version_minor" : 0,
   "format_version_patch" : 0,
   "name"                 : "Command",
   "section"              : 7000,
   "comment"              : "Customer commands section",
   "commands"             : [
      {
         "id"            : 1,
         "name"          : "sensor_reboot",
         "comment"       : "Performs sensor reboot.",
         "public"        : true,
         "level"         : 0,
         "argument"      : "Dummy value"
      },
      {
         "id"            : 10,
         "name"          : "reset_all_parameters",
         "comment"       : "Resets all parameters to default and restarts the sensor.",
         "public"        : true,
         "level"         : 0,
         "argument"      : "Dummy value"
      },
      {
         "id"            : 11,
         "name"          : "reset_all_parameters_but_keep_IP_settings",
         "comment"       : "Resets all parameters except IP settings.",
         "public"        : true,
         "level"         : 0,
         "argument"      : "Section"
      },
      {
         "id"            : 20,
         "name"          : "sw_crc_check",
         "comment"       : "Performs a crc check",
         "public"        : true,
         "level"         : 0,
         "argument"      : "Dummy value"
      }
   ]
}