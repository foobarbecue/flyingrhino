   #Copyright 2013 Aaron Curtis

   #Licensed under the Apache License, Version 2.0 (the "License");
   #you may not use this file except in compliance with the License.
   #You may obtain a copy of the License at

       #http://www.apache.org/licenses/LICENSE-2.0

   #Unless required by applicable law or agreed to in writing, software
   #distributed under the License is distributed on an "AS IS" BASIS,
   #WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   #See the License for the specific language governing permissions and
   #limitations under the License.

import calendar, datetime, re

def dt2jsts(datetime):
    """
    Given a python datetime, convert to javascript timestamp format (milliseconds since Jan 1 1970).
    Do so with microsecond precision, and without adding any timezone offset.
    """
    return calendar.timegm(datetime.timetuple())*1e3+datetime.microsecond/1e3

def logpath2dt(filepath):
    """
    given a dataflashlog in the format produced by Mission Planner,
    return a datetime which says when the file was downloaded from the APM
    """
    return datetime.datetime.strptime(re.match(r'.*/(.*) .*$',filepath).groups()[0],'%Y-%m-%d %H-%M')