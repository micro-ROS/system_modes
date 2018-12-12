# Legal information in Source Code Files

:zap: __This file is not supposed to be published. It is solely for internal
use!__

## Copyright header
#### Bosch Code

Add a copyright / license header to all source files. Note: copyright holder is
always the legal entity. Associate can be added as author (if written agreement
is available). However, instead of putting legal entitiy as copyright holder,
here we go for an approach similar to the one used by many community projects,
e.g. [Golang copyright
policy](https://golang.org/doc/contribute.html#copyright), i.e.

```c++
// Copyright (c) 2016 - for information on the respective copyright owner
// see the NOTICE file and/or the repository <FIXME-repository-address>.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
```
with the [NOTICE](NOTICE) file listing all copyright holders.

#### Third-party code

If an entire source code file is from a third party, leave all
copyright/license information as is or add appropriate copyright/license
information, e.g.

```c++
// This source code is from Awesome Project V 0.9
//   (https://github.com/awesome/project/tree/v0.9
// Copyright (c) 2012-2014 Awesome Inc.
// This source code is licensed under the MIT license found in the
// 3rd-party-licenses.txt file in the root directory of this source tree.
```
I.e. treat as unmodified 3rd-party component.

## Snippet documentation
#### Bosch Code with snippets

Copyright header as for Bosch Code above.

Ensure snippets are commented properly including the following information:
origin (project, version, link), copyright holder, license (alternatively: add
snippet in an atomic commit with the license information there and add required
legal information to the 3rd-party-licenses.txt)

```c++
// Copyright (c) 2016 - for information on the respective copyright owner
// see the NOTICE file and/or the repository <FIXME-repository-address>.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <this_and_that>

// The following class/fct/snippet is from Awesome Project V 0.9
//   (https://github.com/awesome/project/tree/v0.9
// Copyright (c) 2012-2014 Awesome Inc., licensed under the MIT license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
void awesome_function() {
    return;
}

...
```

#### Third-party code with Bosch snippets

As we are creating a derived work licensed under the Apache-2.0 license, 
add the usual copyright header and information on the source:

```c++
// Copyright (c) 2016 - for information on the respective copyright owner
// see the NOTICE file and/or the repository <FIXME-repository-address>.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This source code is derived from Awesome Project V 0.9
//   (https://github.com/awesome/project/tree/v0.9
// Copyright (c) 2012-2014 Awesome Inc., licensed under the MIT license,
// cf. 3rd-party-licenses.txt file in the root directory of this source tree.
```

Again: it would be best if the original version is committed as a first, then
the Bosch changes as follow-up commit.
