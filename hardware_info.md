# Linux Hardware Information Report

- Generated: 2026-10-30 10:03:53
- Hostname: [hostname redacted]


## Operating System

```text
PRETTY_NAME="Ubuntu 22.04.5 LTS"
NAME="Ubuntu"
VERSION_ID="22.04"
VERSION="22.04.5 LTS (Jammy Jellyfish)"
VERSION_CODENAME=jammy
ID=ubuntu
ID_LIKE=debian
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
UBUNTU_CODENAME=jammy

Linux [hostname redacted] 5.15.0-1103-realtime #112-Ubuntu SMP PREEMPT_RT Thu Mar 19 18:04:11 UTC 2026 x86_64 x86_64 x86_64 GNU/Linux
```

## CPU Information

```text
Architecture:                            x86_64
CPU op-mode(s):                          32-bit, 64-bit
Address sizes:                           39 bits physical, 48 bits virtual
Byte Order:                              Little Endian
CPU(s):                                  16
On-line CPU(s) list:                     0-15
Vendor ID:                               GenuineIntel
Model name:                              12th Gen Intel(R) Core(TM) i7-1260P
CPU family:                              6
Model:                                   154
Thread(s) per core:                      2
Core(s) per socket:                      12
Socket(s):                               1
Stepping:                                3
CPU max MHz:                             4700.0000
CPU min MHz:                             400.0000
BogoMIPS:                                4992.00
Flags:                                   fpu vme de pse tsc msr pae mce cx8 apic sep mtrr pge mca cmov pat pse36 clflush dts acpi mmx fxsr sse sse2 ss ht tm pbe syscall nx pdpe1gb rdtscp lm constant_tsc art arch_perfmon pebs bts rep_good nopl xtopology nonstop_tsc cpuid aperfmperf tsc_known_freq pni pclmulqdq dtes64 monitor ds_cpl vmx smx est tm2 ssse3 sdbg fma cx16 xtpr pdcm pcid sse4_1 sse4_2 x2apic movbe popcnt tsc_deadline_timer aes xsave avx f16c rdrand lahf_lm abm 3dnowprefetch cpuid_fault epb invpcid_single ssbd ibrs ibpb stibp ibrs_enhanced tpr_shadow vnmi flexpriority ept vpid ept_ad fsgsbase tsc_adjust bmi1 avx2 smep bmi2 erms invpcid rdseed adx smap clflushopt clwb intel_pt sha_ni xsaveopt xsavec xgetbv1 xsaves split_lock_detect avx_vnni dtherm ida arat pln pts hwp hwp_notify hwp_act_window hwp_epp hwp_pkg_req umip pku ospke waitpkg gfni vaes vpclmulqdq rdpid movdiri movdir64b fsrm md_clear serialize arch_lbr flush_l1d arch_capabilities ibpb_exit_to_user
Virtualization:                          VT-x
L1d cache:                               448 KiB (12 instances)
L1i cache:                               640 KiB (12 instances)
L2 cache:                                9 MiB (6 instances)
L3 cache:                                18 MiB (1 instance)
NUMA node(s):                            1
NUMA node0 CPU(s):                       0-15
Vulnerability Gather data sampling:      Not affected
Vulnerability Indirect target selection: Not affected
Vulnerability Itlb multihit:             Not affected
Vulnerability L1tf:                      Not affected
Vulnerability Mds:                       Not affected
Vulnerability Meltdown:                  Not affected
Vulnerability Mmio stale data:           Not affected
Vulnerability Reg file data sampling:    Mitigation; Clear Register File
Vulnerability Retbleed:                  Not affected
Vulnerability Spec rstack overflow:      Not affected
Vulnerability Spec store bypass:         Mitigation; Speculative Store Bypass disabled via prctl and seccomp
Vulnerability Spectre v1:                Mitigation; usercopy/swapgs barriers and __user pointer sanitization
Vulnerability Spectre v2:                Mitigation; Enhanced / Automatic IBRS; IBPB conditional; PBRSB-eIBRS SW sequence; BHI BHI_DIS_S
Vulnerability Srbds:                     Not affected
Vulnerability Tsa:                       Not affected
Vulnerability Tsx async abort:           Not affected
Vulnerability Vmscape:                   Mitigation; IBPB before exit to userspace
```

## Memory Information

```text
               total        used        free      shared  buff/cache   available
Mem:            30Gi        10Gi       3.7Gi       2.4Gi        16Gi        18Gi
Swap:          2.0Gi       1.7Gi       286Mi
```

## GPU Information

```text
NVIDIA GPU information unavailable.

00:02.0 VGA compatible controller: Intel Corporation Alder Lake-P Integrated Graphics Controller (rev 0c)
```

## Disk Information

```text
NAME          SIZE TYPE FSTYPE   MOUNTPOINT                          MODEL
loop0           4K loop squashfs /snap/bare/5                        
loop1         616M loop squashfs /snap/gnome-46-2404/177             
loop2        13.1M loop squashfs /snap/canonical-livepatch/414       
loop3       264.8M loop squashfs /snap/vlc/4472                      
loop4          12M loop squashfs /snap/snap-store/1447               
loop5        55.5M loop squashfs /snap/core18/2999                   
loop6        55.5M loop squashfs /snap/core18/3084                   
loop8        63.8M loop squashfs /snap/core20/2866                   
loop9          74M loop squashfs /snap/core22/2437                   
loop10         74M loop squashfs /snap/core22/2955                   
loop11       63.8M loop squashfs /snap/core20/2922                   
loop12       66.8M loop squashfs /snap/core24/1643                   
loop13       13.2M loop squashfs /snap/canonical-livepatch/418       
loop14      519.7M loop squashfs /snap/code/273                      
loop15      531.4M loop squashfs /snap/gnome-42-2204/247             
loop16      531.5M loop squashfs /snap/gnome-42-2204/263             
loop17      529.8M loop squashfs /snap/code/268                      
loop18      614.5M loop squashfs                                     
loop19       91.7M loop squashfs /snap/gtk-common-themes/1535        
loop20        395M loop squashfs /snap/mesa-2404/1165                
loop21        402M loop squashfs /snap/mesa-2404/1839                
loop22      262.3M loop squashfs /snap/firefox/9036                  
loop23       12.2M loop squashfs /snap/snap-store/1216               
loop24       50.3M loop squashfs /snap/snapd/27738                   
loop25      615.3M loop squashfs /snap/gnome-46-2404/168             
loop26        828K loop squashfs /snap/snapd-desktop-integration/387 
loop27        828K loop squashfs /snap/snapd-desktop-integration/391 
loop28      321.1M loop squashfs /snap/vlc/3777                      
loop29       66.8M loop squashfs /snap/core24/2124                   
loop30        1.6G loop squashfs /snap/kf6-core24/64                 
loop31       44.7M loop squashfs /snap/snapd/28254                   
loop32      283.8M loop squashfs /snap/firefox/9057                  
nvme0n1     953.9G disk                                              QUANXING P300 1TB
├─nvme0n1p1   512M part vfat     /boot/efi                           
└─nvme0n1p2 953.4G part ext4     /                                   
```

## Filesystem Usage

```text
Filesystem      Size  Used Avail Use% Mounted on
/dev/nvme0n1p2  938G  265G  626G  30% /
/dev/nvme0n1p1  511M  6.1M  505M   2% /boot/efi
```

## Motherboard Information

```text
Requires root permission: sudo dmidecode -t baseboard -t bios
```

## Network Information

Host-specific interface identifiers and network addresses are omitted from this shared report.

## Kernel Information

```text
5.15.0-1103-realtime
```

## Virtualization Information

```text
none
```
