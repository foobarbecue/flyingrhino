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

import calendar, datetime, re, unicodedata, random

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
    try:
        return datetime.datetime.strptime(filepath.split('/')[-1].replace('log','').replace('tlog','').replace(' ','-')[:16],'%Y-%m-%d-%H-%M')
    except AttributeError:
        return datetime.datetime.strptime(re.match(r'.*/(.*) .*$',filepath.replace('_',' ')).groups()[0],'%Y-%m-%d %H-%M')

def slugify(value):
    """
    Copied from django.utils.text.slugify and modified to remove dependencies
    
    Converts to lowercase, removes non-word characters (alphanumerics and
    underscores) and converts spaces to hyphens. Also strips leading and
    trailing whitespace.
    """
    value = unicodedata.normalize('NFKD', unicode(value)).encode('ascii', 'ignore').decode('ascii')
    value = re.sub('[^\w\s-]', '', value).strip().lower()
    return re.sub('[-\s]+', '-', value)

def stringify4sql(tstamp):
    try:
        return tstamp.isoformat().replace('T',' ')
    except AttributeError:
        return str(random.randint(0,1e9))

def start_of_week(dt):
    isocaldt=dt.isocalendar()
    dt=dt-datetime.timedelta(days=dt.weekday())
    return datetime.datetime(year=dt.year, month=dt.month, day=dt.day)

def logpath2week_epoch(filepath):
    return start_of_week(logpath2dt(filepath))