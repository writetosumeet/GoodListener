
char* index_start_str = "<html>\
\
<head>\
<title>Title</title>\
<style>\
table {width:70%;text-align:center;margin-left: auto;margin-right: auto;}\
h2   {color: darkred;}\
tr    {background-color:ivory;}\
th    {padding:15px;background-color:darksalmon;color:ivory;}\
td    {padding:2px;}\
</style>\
</head>\
\
<body style=\"font-family: verdana;\">\
\
<p style=\"width:100%;text-align:right;\">\
<a href=\"/settings.html\"><b>Settings</b></a>&nbsp;&nbsp;\
</p>\
\
<table style=\"width:100%;text-align:center;margin-left: auto;margin-right: auto;\">\
<tr style=\"background-color:white;\"><td style=\"width:15%\"><H2>Recordings</H2></td>\
<td style=\"width:22%\">&nbsp;</td>\
<td style=\"width:16%\">&nbsp;</td>\
<td style=\"width:14%;\"><b><a href=\"/ring_all\" download=\"ring.ogg\">Download All</a></b></td>\
<td style=\"width:17%;\"><audio controls title=\"ring.ogg\"><source src=\"/ring_all\" type=\"audio/ogg\">Player unsupported</audio></td>\
<td style=\"width:16%\">&nbsp;</td></tr>\
</table>\
\
<table>\
<tr>\
<th style=\"width:40%;\">Filename</th>\
<th style=\"width:20%;\">Size (bytes)</th>\
<th style=\"width:20%;\">Download</th>\
<th style=\"width:20%;\">Play</th>\
</tr>";

char* index_end_str = "</table>\
\
<br>\
\
</body>\
</html>";


char* settings_start_str = "<html>\
\
<head>\
<title>Title</title>\
<style>\
table {width:80%;}\
h2   {color: darkred;}\
tr    {background-color:ivory;}\
th    {padding:15px;background-color:darksalmon;color:ivory;}\
td    {padding:5px;}\
</style>\
  </head>\
  \
  <body style=\"font-family: verdana;\">\
  \
  <p style=\"width:100%;text-align:right;\">\
  <a href=\"/index.html\"><b>Back to Recordings</b></a>&nbsp;&nbsp;  \
  </p>\
  \
  <H2>WiFi Settings</H2>\
  <table>\
  <tr>\
  <td style=\"width:30%;text-align:right;\">esp ssid&nbsp;&nbsp;</td>\
  <td style=\"width:50%;text-align:left;\">&nbsp;&nbsp;espssid</td>\
  <td style=\"width:20%\">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</td>\
  </tr>\
  <tr>\
  <td style=\"text-align:right;\">esp password&nbsp;&nbsp;</td>\
  <td style=\"text-align:left;\">&nbsp;&nbsp;esppassword</td>\
  <td>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</td>\
  </tr>\
  <tr>\
  <td colspan=\"2\">&nbsp;&nbsp;</td>\
  <td style=\"text-align:center;\">update button</td>\
  </tr>\
  </table>\
  <br>\
  <table>                                                            \
  <tr>                                                               \
  <td style=\"width:30%;text-align:right;\">router ssid&nbsp;&nbsp;</td>\
  <td style=\"width:50%;text-align:left;\">&nbsp;&nbsp;routerssid</td>  \
  <td style=\"width:20%\">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</td>  \
  </tr>                                                              \
  <tr>                                                               \
  <td style=\"text-align:right;\">router password&nbsp;&nbsp;</td>      \
  <td style=\"text-align:left;\">&nbsp;&nbsp;routerpassword</td>        \
  <td>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</td>                      \
  </tr>                                                              \
  <tr>                                                               \
  <td colspan=\"2\">&nbsp;&nbsp;</td>                                \
  <td style=\"text-align:center;\">update button</td>                \
  </tr>                                                              \
  </table>                                                           \
  <br>                                                               \
  \
  <H2>Radio Link</H2>\
  <table>\
  <tr>\
  <td style=\"width:30%;text-align:right;\">URL&nbsp;&nbsp;</td>\
  <td style=\"width:50%;text-align:left;\">&nbsp;&nbsp;<u>url</u></td>\
  <td style=\"width:20%\">&nbsp;&nbsp;</td>\
  </tr>\
  <tr>\
  <td colspan=\"2\">&nbsp;&nbsp;</td>\
  <td style=\"text-align:center;\">update button</td>\
  </tr>\
  </table>\
  <br>\
  \
  <H2>Upgrade Software</H2>\
  <table>\
  <tr>\
  <td style=\"width:30%;text-align:right;\">Current version&nbsp;&nbsp;</td>\
  <td style=\"width:50%;text-align:left;\">&nbsp;&nbsp;v1.0</td>\
  <td style=\"width:20%\">&nbsp;&nbsp;</td>\
  </tr>\
  <tr>\
  <td style=\"text-align:right;\">Latest version&nbsp;&nbsp;</td>\
  <td style=\"text-align:left;\">&nbsp;&nbsp;v1.0</td>\
  <td>&nbsp;&nbsp;</td>\
  </tr>\
  <tr>\
  <td colspan=\"2\">&nbsp;&nbsp;</td>\
  <td style=\"text-align:center;\">update button</td>\
  </tr>\
  </table>\
  <br>\
  \
  </body>\
  </html>";






